using Gurobi, Base.Test, MathOptInterface, MathOptInterfaceTests, MathOptInterfaceGurobi

const MOIT = MathOptInterfaceTests
const MOIGRB = MathOptInterfaceGurobi

@testset "MathOptInterfaceGurobi" begin
    @testset "Linear tests" begin
        linconfig = MOIT.TestConfig(1e-8,1e-8,true,true,true)
        solver = MOIGRB.MOIGurobiSolver(OutputFlag=0)
        MOIT.contlineartest(solver , linconfig, ["linear10","linear12","linear8a","linear8b","linear8c"])
        
        solver_nopresolve = MOIGRB.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)
        MOIT.contlineartest(solver_nopresolve , linconfig, ["linear10","linear12","linear8a"])

        linconfig_nocertificate = MOIT.TestConfig(1e-8,1e-8,true,true,false)
        MOIT.linear12test(solver, linconfig_nocertificate)
        MOIT.linear8atest(solver, linconfig_nocertificate)

        # 10 is ranged
    end

    @testset "Quadratic tests" begin
        quadconfig = MOIT.TestConfig(1e-5,1e-3,false,false,true)
        solver = MOIGRB.MOIGurobiSolver(OutputFlag=0)
        MOIT.contquadratictest(solver, quadconfig)
    end

    @testset "Linear Conic tests" begin
        linconfig = MOIT.TestConfig(1e-8,1e-8,true,true,true)
        solver = MOIGRB.MOIGurobiSolver(OutputFlag=0)
        MOIT.lintest(solver, linconfig, ["lin3","lin4"])

        solver_nopresolve = MOIGRB.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)
        MOIT.lintest(solver_nopresolve, linconfig)
    end

    @testset "Integer Linear tests" begin
        intconfig = MOIT.TestConfig(1e-8,1e-8,true,true,true)
        solver = MOIGRB.MOIGurobiSolver(OutputFlag=0)
        MOIT.intlineartest(solver, intconfig, ["int3"])

        # 3 is ranged
    end
end
;
