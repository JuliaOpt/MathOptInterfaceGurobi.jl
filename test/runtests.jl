using Gurobi, Base.Test, MathOptInterface, MathOptInterfaceTests, MathOptInterfaceGurobi

const MOIT = MathOptInterfaceTests
const MOIGRB = MathOptInterfaceGurobi

@testset "MathOptInterfaceGurobi" begin
    @testset "Linear tests" begin
        linconfig = linconfig = MOIT.TestConfig()
        solverf() = GurobiSolverInstance(OutputFlag=0)
        MOIT.contlineartest(solverf, linconfig, ["linear10","linear12","linear8a","linear8b","linear8c"])
        
        solverf_nopresolve() = GurobiSolverInstance(OutputFlag=0, InfUnbdInfo=1)
        MOIT.contlineartest(solverf_nopresolve, linconfig, ["linear10","linear12","linear8a"])

        linconfig_nocertificate = MOIT.TestConfig(infeas_certificates=false)
        MOIT.linear12test(solverf, linconfig_nocertificate)
        MOIT.linear8atest(solverf, linconfig_nocertificate)

        # 10 is ranged
    end

    @testset "Quadratic tests" begin
        quadconfig = MOIT.TestConfig(atol=1e-4, rtol=1e-4, duals=false, query=false)
        solverf() = GurobiSolverInstance(OutputFlag=0)
        MOIT.contquadratictest(solverf, quadconfig)
    end

    @testset "Linear Conic tests" begin
        linconfig = MOIT.TestConfig()
        solverf() = GurobiSolverInstance(OutputFlag=0)
        MOIT.lintest(solverf, linconfig, ["lin3","lin4"])

        solverf_nopresolve() = GurobiSolverInstance(OutputFlag=0, InfUnbdInfo=1)
        MOIT.lintest(solverf_nopresolve, linconfig)
    end

    @testset "Integer Linear tests" begin
        intconfig = MOIT.TestConfig()
        solverf() = GurobiSolverInstance(OutputFlag=0)
        MOIT.intlineartest(solverf, intconfig, ["int3"])

        # 3 is ranged
    end
end
;
