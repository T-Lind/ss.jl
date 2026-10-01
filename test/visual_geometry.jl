using Test
using SatelliteSim

@testset "visual geometry contracts" begin
    for n in (1, 2, 3, 5, 6, 9, 13, 20, 27, 33, 37, 60)
        pts = SatelliteSim._cluster(n, 3.6, 0.945)
        @test length(pts) == n
        @test all(p -> p[3] > 0 && hypot(p[1],p[2]) + p[3] <= 3.6 + 1e-12, pts)
        @test all(hypot(pts[i][1]-pts[j][1],pts[i][2]-pts[j][2]) >= pts[i][3]+pts[j][3]
                  for i in eachindex(pts) for j in 1:(i-1))
    end
    pts = SatelliteSim._cluster(33,3.6,0.945)
    @test [count(p -> isapprox(hypot(p[1],p[2]),r; atol=1e-10),pts)
           for r in (0.18*3.6,0.47*3.6,0.83*3.6)] == [3,10,20]
    for n in (1,6,9,13,33)
        m,s = rocket_mesh(diameters=[9.0,9.0],prop_masses=[3400000.0,1200000.0],
                          n_engines=[n,6],fairing=false)
        bell = SatelliteSim._cluster(n,3.6,0.945)[1]
        exit = 0.72 - 4bell[3]
        @test s[1].x0 ≈ exit
        @test minimum(p[1] for t in m.tris[s[1].t0:s[1].t1] for p in t) ≈ exit
        @test exit + 4bell[3] ≈ 0.72 # throat is above the plate
    end
    for d in (2.1,4.2,8.4)
        parts,h = SatelliteSim._lander_payload_mesh(d)
        @test all(mesh_volume(p) > 0 for p in parts)
        points = [v for p in parts for t in p.tris for v in t]
        @test minimum(p[1] for p in points) ≈ 0 atol=1e-10
        @test maximum(hypot(p[2],p[3]) for p in points) <= d/2 + 1e-10
        @test maximum(p[1] for p in points) ≈ h atol=1e-10
    end
end
