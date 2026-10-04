using Test, SatelliteSim

# Branches in the same first-stage attachment lifecycle. These also feed the
# JS/Julia parity fixture, so mass and thrust are checked in both runtimes.
function staging_cases()
    results = []
    for (name, prop, delay, sep) in (("burning",10000.0,0.0,0.0), ("waiting",10000.0,16.0,0.0),
                                     ("spent",400.0,0.0,100.0), ("gone",400.0,0.0,0.0))
        core = Stage(:core,1000.0,5000.0,1e6,300.0,0.0)
        upper = Stage(:upper,300.0,1500.0,80000.0,300.0,0.0)
        side = Stage(:side,200.0,prop,200000.0,300.0,0.0)
        lv = LaunchVehicle(stages=[core,upper],fairing_mass=0.0,payload_mass=100.0,sref=1.0,cd=SatelliteSim.LV_CD_TABLE,
                           boosters=[BoosterSet(stage=side,count=2,ignition_delay=delay,sep_delay=sep)])
        r = simulate_ascent(lv, AscentGuidance();dt=.1,log_dt=.01,t_max=22.0)
        push!(results,(name,r))
    end
    results
end

@testset "boosters leave with their first-stage attachment" begin
    for (name,r) in staging_cases()
        cs = only(filter(e->e.name===:sep_core,r.events))
        bs = only(filter(e->e.name===:sep_side,r.events))
        @test isapprox(cs.t,5000*G0*300/1e6;atol=1e-8)
        @test bs.t <= cs.t
        name != "gone" && (@test bs.t == cs.t)
        last = filter(e->e.t==cs.t,r.events)[end]
        @test isapprox(last.m,1900.0;atol=1e-6)
        name=="waiting" && (@test !any(e->e.name===:ignition_side,r.events))
        name=="burning" && (@test !any(e->e.name===:burnout_side,r.events))
        gap = findall(t->cs.t<=t<cs.t+4,r.log.t)
        @test length(gap)>1
        @test all(i->r.log.thrust[i]==0.0,gap)
        @test all(i->isapprox(r.log.m[i],1900.0;atol=1e-6),gap)
        @test any(e->e.name===:ignition_upper,r.events)
    end
end

@testset "mission cutoff leaves attached booster mass but stops thrust" begin
    core=Stage(:core,1000.0,5000.0,1e6,300.0,0.0)
    side=Stage(:side,200.0,10000.0,200000.0,300.0,0.0)
    lv=LaunchVehicle(stages=[core],fairing_mass=0.0,payload_mass=100.0,sref=1.0,
                     cd=SatelliteSim.LV_CD_TABLE,boosters=[BoosterSet(stage=side,count=2)])
    r=simulate_ascent(lv,AscentGuidance(cutoff=:apogee,apogee_target=500.0);dt=.1,t_max=22.0)
    @test r.events[end].name===:seco
    @test r.log.thrust[end]==0
    @test !any(e->e.name===:sep_side,r.events)
    @test r.m>100+2*200
end
