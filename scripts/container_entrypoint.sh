#!/bin/sh
set -eu

# Existing Render services may retain the old `auto` environment override.
# Host CPU affinity does not describe a fractional container CPU quota.
case "${JULIA_NUM_THREADS:-auto}" in
  auto|'') export JULIA_NUM_THREADS=2 ;;
esac
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"

exec julia --project=. --compiled-modules=existing \
  --heap-size-hint="${SSJL_HEAP_SIZE_HINT:-256M}" "$@"
