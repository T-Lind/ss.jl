# Fixed-step classical RK4 with preallocated work buffers.
#
# Step sizes are phase-scheduled by the simulation driver (coarse in orbit,
# fine through entry). Extension point: swap `rk4_step!` for an adaptive
# scheme (e.g. Dormand-Prince) behind the same interface if higher-order
# accuracy or automatic step control is needed.

struct RK4Work
    k1::Vector{Float64}
    k2::Vector{Float64}
    k3::Vector{Float64}
    k4::Vector{Float64}
    xt::Vector{Float64}
    RK4Work(n::Int) = new(zeros(n), zeros(n), zeros(n), zeros(n), zeros(n))
end

"""
    rk4_step!(xout, x, t, dt, w, scn, ctx)

One RK4 step of `dynamics!` from (x, t) to (xout, t+dt). `xout` may alias `x`.
"""
function rk4_step!(xout::Vector{Float64}, x::Vector{Float64}, t::Float64, dt::Float64,
                   w::RK4Work, scn::Scenario, ctx::FlightContext)
    n = length(x)
    dynamics!(w.k1, x, scn, ctx, t)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k1[i]; end
    dynamics!(w.k2, w.xt, scn, ctx, t + 0.5dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + 0.5dt * w.k2[i]; end
    dynamics!(w.k3, w.xt, scn, ctx, t + 0.5dt)
    @inbounds for i in 1:n; w.xt[i] = x[i] + dt * w.k3[i]; end
    dynamics!(w.k4, w.xt, scn, ctx, t + dt)
    @inbounds for i in 1:n
        xout[i] = x[i] + (dt / 6) * (w.k1[i] + 2w.k2[i] + 2w.k3[i] + w.k4[i])
    end
    return nothing
end
