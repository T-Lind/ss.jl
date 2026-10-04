# Compact parity reference for attachment transitions and the short-burn lander.
using SatelliteSim
const S=SatelliteSim
num(x)=isfinite(Float64(x)) ? string(Float64(x)) : "null"
arr(io,x)=print(io,"[",join(num.(x),","),"]")
open(ARGS[1],"w") do io
    print(io,"{\"staging\":[")
    for (i,(name,prop,delay,sep)) in enumerate((("burning",10000.0,0.0,0.0),("waiting",10000.0,16.0,0.0),("spent",400.0,0.0,100.0),("gone",400.0,0.0,0.0)))
        core=Stage(:core,1000.0,5000.0,1e6,300.0,0.0)
        upper=Stage(:upper,300.0,1500.0,80000.0,300.0,0.0)
        side=Stage(:side,200.0,prop,200000.0,300.0,0.0)
        lv=LaunchVehicle(stages=[core,upper],fairing_mass=0.0,payload_mass=100.0,sref=1.0,cd=SatelliteSim.LV_CD_TABLE,
                         boosters=[BoosterSet(stage=side,count=2,ignition_delay=delay,sep_delay=sep)])
        r=simulate_ascent(lv,AscentGuidance();dt=.1,log_dt=.01,t_max=22.0)
        i>1&&print(io,",")
        print(io,"{\"name\":\"",name,"\",\"events\":[")
        for (j,e) in enumerate(r.events)
            j>1&&print(io,",")
            print(io,"{\"name\":\"",e.name,"\",\"t\":",num(e.t),",\"m\":",num(e.m),"}")
        end
        print(io,"],\"log\":{")
        for (j,k) in enumerate((:t,:m,:thrust))
            j>1&&print(io,",");print(io,"\"",k,"\":");arr(io,getfield(r.log,k))
        end
        print(io,"}}")
    end
    print(io,"],\"light\":[")
    r=(R_MOON+15291.055387647589,0.0,0.0);v=(.148114443,1692.33544,0.0);m=3344.6766118550195
    for (i,thr) in enumerate((.1,.02))
        l=Lander(mdry=100.0,mprop=m-100,thrust=45000.0,isp=311.0,throttle_min=thr)
        d=powered_descent(l,r,v,m;h_gate=2000.0)
        i>1&&print(io,",");print(io,"{\"outcome\":\"",d.outcome,"\"")
        for k in (:t_touchdown,:t_gate,:v_vertical,:v_horizontal,:prop_left,:pitch0,:pitch_rate)
            print(io,",\"",k,"\":",num(getfield(d,k)))
        end
        print(io,"}")
    end
    print(io,"]}")
end
