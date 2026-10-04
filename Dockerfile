FROM julia:1.12-bookworm

WORKDIR /app

# The package is stdlib-only. Copy the numerical core first so its compile
# cache survives page-only edits, then add the server and browser assets.
COPY Project.toml ./
COPY src/ ./src/
RUN julia --project=. -e "using Pkg; Pkg.instantiate(); using SatelliteSim"

COPY scripts/ ./scripts/

ENV HOST=0.0.0.0 \
    PORT=10000 \
    JULIA_NUM_THREADS=2 \
    OPENBLAS_NUM_THREADS=1
EXPOSE 10000

# Leave room for native code and libraries on the 512 MiB free instance.
# The entry point also clamps older Render environments still set to `auto`.
ENTRYPOINT ["sh", "scripts/container_entrypoint.sh"]
CMD ["scripts/panel.jl"]
