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
    JULIA_NUM_THREADS=auto
EXPOSE 10000

CMD ["julia", "--project=.", "--compiled-modules=existing", "-t", "auto", "scripts/panel.jl"]