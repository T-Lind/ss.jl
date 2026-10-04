#!/usr/bin/env bash
set -euo pipefail

# No mounted cache or startup override: test the image Render actually runs.
docker run -d --name ssjl-ci --memory=512m --memory-swap=512m --cpus=0.1 \
  -e JULIA_NUM_THREADS=auto \
  -p 127.0.0.1:10000:10000 ssjl:ci

ready=false
for attempt in $(seq 1 90); do
  if curl --silent --fail --max-time 3 http://127.0.0.1:10000/api/health > /tmp/ssjl-health.json; then
    jq -e '.ok == true and .threads == 2' /tmp/ssjl-health.json
    ready=true
    break
  fi
  if [ "$(docker inspect ssjl-ci --format '{{.State.Running}}')" != true ]; then
    echo 'Container exited before becoming healthy' >&2
    exit 1
  fi
  sleep 2
done
[ "$ready" = true ] || { echo 'Cold HTTP readiness timed out' >&2; exit 1; }

for path in / /build /launch /api/catalogue; do
  curl --silent --show-error --fail --max-time 15 "http://127.0.0.1:10000$path" > /dev/null
done

# The first real mission is deliberately cold. Validate API semantics after
# compilation, not just that the port accepts a TCP connection.
curl --silent --show-error --fail --max-time 180 \
  --data 'mode=orbit&orbit=leo' http://127.0.0.1:10000/api/run > /tmp/ssjl-mission.json &
mission_pid=$!
sleep 2
curl --silent --show-error --fail --max-time 30 http://127.0.0.1:10000/api/health | jq -e '.ok == true'
wait "$mission_pid"
jq -e '.ok == true and .mode == "orbit"' /tmp/ssjl-mission.json
curl --silent --show-error --fail --max-time 10 http://127.0.0.1:10000/api/health | jq -e '.ok == true'
