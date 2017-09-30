module GurobiMathOptInterface
# todo
# single get/set
# fix chg coeff
# inplace getters
# updates!

export MOIGurobiSolver

const GRBMOI = GurobiMathOptInterface

import Base.show, Base.copy

# Standard LP interface
# importall MathProgBase.SolverInterface

using Gurobi
const GRB = Gurobi
using MathOptInterface
const MOI = MathOptInterface
using LinQuadOptInterface
const LQOI = LinQuadOptInterface


struct MOIGurobiSolver <: LQOI.LinQuadSolver
    options
end
function MOIGurobiSolver(;kwargs...)
    return MOIGurobiSolver(kwargs)
end

import Gurobi.Model

function GRB.Model(env::GRB.Env)
    return GRB.Model(env,"defaultname")
end

mutable struct GurobiSolverInstance <: LQOI.LinQuadSolverInstance
    
    LQOI.@LinQuadSolverInstanceBase
    env
    
end

function MOI.SolverInstance(s::MOIGurobiSolver)

    env = GRB.Env()
    m = GurobiSolverInstance(
        (LQOI.@LinQuadSolverInstanceBaseInit)...,
        env
    )
    for (name,value) in s.options
        GRB.setparam!(env, GRB.GRB_CONTROLS_DICT[name], value)
    end
    # csi.inner.mipstart_effort = s.mipstart_effortlevel
    # if s.logfile != ""
    #     LQOI.lqs_setlogfile!(env, s.logfile)
    # end
    return m
end

#=
    inner wrapper
=#

#=
    Main
=#

# LinQuadSolver # Abstract type
# done above

# LQOI.lqs_setparam!(env, name, val)
# TODO fix this one
LQOI.lqs_setparam!(m::GurobiSolverInstance, name, val) = GRB.setparam!(m.env, GRB.GRB_CONTROLS_DICT[name], val)

# LQOI.lqs_setlogfile!(env, path)
# TODO fix this one
LQOI.lqs_setlogfile!(m::GurobiSolverInstance, path) = GRB.setlogfile(m.env, path::String)

# LQOI.lqs_getprobtype(m)
# TODO - consider removing, apparently useless

#=
    Constraints
=#

cintvec(v::Vector) = convert(Vector{Int32}, v)
cdoublevec(v::Vector) = convert(Vector{Float64}, v)

_getsense(m::GurobiSolverInstance, ::MOI.EqualTo{Float64}) = Cchar('=')
_getsense(m::GurobiSolverInstance, ::MOI.LessThan{Float64}) = Cchar('<')
_getsense(m::GurobiSolverInstance, ::MOI.GreaterThan{Float64}) = Cchar('>')
_getsense(m::GurobiSolverInstance, ::MOI.Zeros)        = Cchar('=')
_getsense(m::GurobiSolverInstance, ::MOI.Nonpositives) = Cchar('<')
_getsense(m::GurobiSolverInstance, ::MOI.Nonnegatives) = Cchar('>')
_getboundsense(m::GurobiSolverInstance, ::MOI.Nonpositives) = Cchar('>')
_getboundsense(m::GurobiSolverInstance, ::MOI.Nonnegatives) = Cchar('<')

# LQOI.lqs_chgbds!(m, colvec, valvec, sensevec)
# TODO - improve single type
function LQOI.lqs_chgbds!(m::GRB.Model, colvec, valvec, sensevec)
    lb_len = count(x->x==Cchar('L'), sensevec)
    LB_val = Array{Float64}(0)
    sizehint!(LB_val, lb_len)
    LB_col = Array{Cint}(0)
    sizehint!(LB_col, lb_len)
    
    ub_len = count(x->x==Cchar('U'), sensevec)
    UB_val = Array{Float64}(0)
    sizehint!(UB_val, ub_len)
    UB_col = Array{Cint}(0)
    sizehint!(UB_col, ub_len)

    for i in eachindex(valvec)
        if sensevec[i] == Cchar('L')
            push!(LB_col, colvec[i])
            push!(LB_val, valvec[i])
        elseif sensevec[i] == Cchar('U')
            push!(UB_col, colvec[i])
            push!(UB_val, valvec[i])
        end
    end

    if lb_len > 0
        GRB.set_dblattrlist!(m, "LB", LB_col, LB_val)
    end
    
    if ub_len > 0
        GRB.set_dblattrlist!(m, "UB", UB_col, UB_val)
    end

    GRB.update_model!(m)
    nothing
end


# LQOI.lqs_getlb(m, col)
LQOI.lqs_getlb(m::GRB.Model, col) = (GRB.update_model!(m);GRB.get_dblattrlist( m, "LB", GRB.ivec(col))[1])
# LQOI.lqs_getub(m, col)
LQOI.lqs_getub(m::GRB.Model, col) = GRB.get_dblattrlist( m, "UB", GRB.ivec(col))[1]

# LQOI.lqs_getnumrows(m)
LQOI.lqs_getnumrows(m::GRB.Model) = GRB.num_constrs(m)

# LQOI.lqs_addrows!(m, rowvec, colvec, coefvec, sensevec, rhsvec)
LQOI.lqs_addrows!(m::GRB.Model, rowvec, colvec, coefvec, sensevec, rhsvec) = (GRB.add_constrs!(m::GRB.Model, rowvec, colvec, coefvec, sensevec, rhsvec);GRB.update_model!(m))

# LQOI.lqs_getrhs(m, rowvec)
LQOI.lqs_getrhs(m::GRB.Model, row) = GRB.get_dblattrlist( m, "RHS", GRB.ivec(row))[1]

# colvec, coef = LQOI.lqs_getrows(m, rowvec)
# TODO improve
function LQOI.lqs_getrows(m::GRB.Model, idx)
    A = GRB.get_constrs(m, idx, 1)'
    return A.rowval-1, A.nzval
end

# LQOI.lqs_getcoef(m, row, col) #??
# TODO improve
function LQOI.lqs_getcoef(m::GRB.Model, row, col) #??
    return getcoeff(model::Model, row::Integer, col::Integer)
    # A = GRB.get_rows(m, row, row)'
    # cols = A.rowval
    # vals = A.nzval

    # pos = findfirst(cols, col)
    # if pos > 0
    #     return vals[pos]
    # else
    #     return 0.0
    # end
end

# LQOI.lqs_chgcoef!(m, row, col, coef)
# TODO SPLIT THIS ONE
function LQOI.lqs_chgcoef!(m::GRB.Model, row, col, coef) 
    if row == 0
        GRB.set_dblattrlist!(m, "Obj", Cint[col], Float64[coef])
    elseif col == 0
        GRB.set_dblattrlist!(m, "RHS", Cint[row], Float64[coef])
    else
        GRB.chg_coeffs!(m, row, col, coef)
        #TODO fix this function in gurobi
    end
end

# LQOI.lqs_delrows!(m, row, row)
LQOI.lqs_delrows!(m::GRB.Model, rowbeg, rowend) = GRB.del_constrs!(m, cintvec(collect(rowbeg:rowend))) 

# LQOI.lqs_chgctype!(m, colvec, typevec)
# TODO fix types
LQOI.lqs_chgctype!(m::GRB.Model, colvec, typevec) = GRB.set_charattrlist!(m, "VType", GRB.ivec(colvec), GRB.cvec(typevec))

# LQOI.lqs_chgsense!(m, rowvec, sensevec)
# TODO fix types
LQOI.lqs_chgsense!(m::GRB.Model, rowvec, sensevec) = GRB.set_charattrlist!(m, "Sense", GRB.ivec(rowvec), GRB.cvec(sensevec))

const VAR_TYPE_MAP = Dict{Symbol,Cchar}(
    :CONTINUOUS => Cchar('C'),
    :INTEGER => Cchar('I'),
    :BINARY => Cchar('B')
)
LQOI.lqs_vartype_map(m::GurobiSolverInstance) = VAR_TYPE_MAP

# LQOI.lqs_addsos(m, colvec, valvec, typ)
LQOI.lqs_addsos!(m::GRB.Model, colvec, valvec, typ) = (GRB.add_sos!(m, typ, colvec, valvec);GRB.update_model!(m))
# LQOI.lqs_delsos(m, idx, idx)
LQOI.lqs_delsos!(m::GRB.Model, idx1, idx2) = (GRB.del_sos!(m, cintvec(collect(idx1:idx2)));GRB.update_model!(m))

const SOS_TYPE_MAP = Dict{Symbol,Symbol}(
    :SOS1 => :SOS1,#Cchar('1'),
    :SOS2 => :SOS2#Cchar('2')
)
LQOI.lqs_sertype_map(m::GurobiSolverInstance) = SOS_TYPE_MAP

# LQOI.lqs_getsos(m, idx)
# TODO improve getting processes
function LQOI.lqs_getsos(m::GRB.Model, idx)
    A, types = GRB.get_sos_matrix(m::GRB.Model)
    line = A[idx,:] #sparse vec
    cols = line.nzind
    vals = line.nzval
    typ = types[idx] == Cint(1) ? :SOS1 : :SOS2
    return cols, vals, typ
end

# LQOI.lqs_getnumqconstrs(m)
LQOI.lqs_getnumqconstrs(m::GRB.Model) = GRB.num_qconstrs(m)

# LQOI.lqs_addqconstr(m, cols,coefs,rhs,sense, I,J,V)
LQOI.lqs_addqconstr!(m::GRB.Model, cols,coefs,rhs,sense, I,J,V) = GRB.add_qconstr!(m, cols, coefs, I, J, V, sense, rhs)

# LQOI.lqs_chgrngval
LQOI.lqs_chgrngval!(m::GRB.Model, rows, vals) = GRB.chg_rhsrange!(m, cintvec(rows), -vals)

const CTR_TYPE_MAP = Dict{Symbol,Cchar}(
    :RANGE => Cchar('R'),
    :LOWER => Cchar('L'),
    :UPPER => Cchar('U'),
    :EQUALITY => Cchar('E')
)
LQOI.lqs_ctrtype_map(m::GurobiSolverInstance) = CTR_TYPE_MAP

#=
    Objective
=#

# LQOI.lqs_copyquad(m, intvec,intvec, floatvec) #?
function LQOI.lqs_copyquad!(m::GRB.Model, I, J, V)
    GRB.delq!(m)
    GRB.add_qpterms!(m, I, J, V)
    return nothing
end

# LQOI.lqs_chgobj(m, colvec,coefvec)
function LQOI.lqs_chgobj!(m::GRB.Model, colvec, coefvec) 
    nvars = GRB.num_vars(m)
    obj = zeros(Float64, nvars)

    for i in eachindex(colvec)
        obj[colvec[i]] = coefvec[i]
    end

    GRB.set_dblattrarray!(m, "Obj", 1, num_vars(m), obj)
    GRB.update_model!(m)
    nothing
end

# LQOI.lqs_chgobjsen(m, symbol)
# TODO improve min max names
function LQOI.lqs_chgobjsen!(m::GRB.Model, symbol)
    if symbol in [:minimize, :Min]
        GRB.set_sense!(m, :minimize)
    else
        GRB.set_sense!(m, :maximize)
    end
    GRB.update_model!(m)
end
    

# LQOI.lqs_getobj(m)
LQOI.lqs_getobj(m::GRB.Model) = GRB.get_dblattrarray( m, "Obj", 1, num_vars(m)   )

# lqs_getobjsen(m)
function LQOI.lqs_getobjsen(m::GRB.Model)
    s = GRB.model_sense(m)
    if s in [:maximize, :Max]
        return MOI.MaxSense
    else
        return MOI.MinSense
    end
end

#=
    Variables
=#

# LQOI.lqs_getnumcols(m)
LQOI.lqs_getnumcols(m::GRB.Model) = (GRB.update_model!(m); GRB.num_vars(m))

# LQOI.lqs_newcols!(m, int)
LQOI.lqs_newcols!(m::GRB.Model, int) = (GRB.add_cvars!(m, zeros(int));GRB.update_model!(m))

# LQOI.lqs_delcols!(m, col, col)
LQOI.lqs_delcols!(m::GRB.Model, col, col2) = GRB.del_vars!(m, col)

# LQOI.lqs_addmipstarts(m, colvec, valvec)
function LQOI.lqs_addmipstarts!(m::GRB.Model, colvec, valvec) 
    x = zeros(GRB.num_vars(m))
    for i in eachindex(colvec)
        x[colvec[i]] = valvec[i]
    end
    GRB.loadbasis(m, x)
end
#=
    Solve
=#

# LQOI.lqs_mipopt!(m)
LQOI.lqs_mipopt!(m::GRB.Model) = LQOI.lqs_lpopt!(m)

# LQOI.lqs_qpopt!(m)
LQOI.lqs_qpopt!(m::GRB.Model) = LQOI.lqs_lpopt!(m)

# LQOI.lqs_lpopt!(m)
LQOI.lqs_lpopt!(m::GRB.Model) = (GRB.update_model!(m);GRB.write_model(m, "test.lp");GRB.optimize(m))

# LQOI.lqs_terminationstatus(m)
function LQOI.lqs_terminationstatus(model::GurobiSolverInstance)
    m = model.inner 
    return MOI.Success
    # stat_lp = GRB.get_lp_status2(m)
    # if GRB.is_mip(m)
    #     stat_mip = GRB.get_mip_status2(m)
    #     if stat_mip == GRB.MIP_NotLoaded
    #         return MOI.OtherError
    #     elseif stat_mip == GRB.MIP_LPNotOptimal
    #         # MIP search incomplete but there is no linear sol
    #         # return MOI.OtherError
    #         return MOI.InfeasibleOrUnbounded
    #     elseif stat_mip == GRB.MIP_NoSolFound
    #         # MIP search incomplete but there is no integer sol
    #         other = GRBmoi_stopstatus(m)
    #         if other == MOI.OtherError
    #             return MOI.SlowProgress#OtherLimit
    #         else 
    #             return other
    #         end

    #     elseif stat_mip == GRB.MIP_Solution
    #         # MIP search incomplete but there is a solution
    #         other = GRBmoi_stopstatus(m)
    #         if other == MOI.OtherError
    #             return MOI.OtherLimit
    #         else 
    #             return other
    #         end

    #     elseif stat_mip == GRB.MIP_Infeasible
    #         if GRB.hasdualray(m)
    #             return MOI.Success
    #         else
    #             return MOI.InfeasibleNoResult
    #         end
    #     elseif stat_mip == GRB.MIP_Optimal
    #         return MOI.Success
    #     elseif stat_mip == GRB.MIP_Unbounded
    #         if GRB.hasprimalray(m)
    #             return MOI.Success
    #         else
    #             return MOI.UnboundedNoResult
    #         end
    #     end
    #     return MOI.OtherError
    # else
    #     if stat_lp == GRB.LP_Unstarted
    #         return MOI.OtherError
    #     elseif stat_lp == GRB.LP_Optimal
    #         return MOI.Success
    #     elseif stat_lp == GRB.LP_Infeasible
    #         if GRB.hasdualray(m)
    #             return MOI.Success
    #         else
    #             return MOI.InfeasibleNoResult
    #         end
    #     elseif stat_lp == GRB.LP_CutOff
    #         return MOI.ObjectiveLimit
    #     elseif stat_lp == GRB.LP_Unfinished
    #         return GRBmoi_stopstatus(m)
    #     elseif stat_lp == GRB.LP_Unbounded
    #         if GRB.hasprimalray(m)
    #             return MOI.Success
    #         else
    #             return MOI.UnboundedNoResult
    #         end
    #     elseif stat_lp == GRB.LP_CutOffInDual
    #         return MOI.ObjectiveLimit
    #     elseif stat_lp == GRB.LP_Unsolved
    #         return MOI.OtherError
    #     elseif stat_lp == GRB.LP_NonConvex
    #         return MOI.InvalidInstance
    #     end
    #     return MOI.OtherError
    # end
end

function GRBmoi_stopstatus(m::GRB.Model)
    ss = GRB.get_stopstatus(m)
    if ss == GRB.StopTimeLimit
        return MOI.TimeLimit
    elseif ss == GRB.StopControlC
        return MOI.Interrupted
    elseif ss == GRB.StopNodeLimit
        # should not be here
        warn("should not be here")
        return MOI.NodeLimit
    elseif ss == GRB.StopIterLimit
        return MOI.IterationLimit
    elseif ss == GRB.StopMIPGap
        return MOI.ObjectiveLimit
    elseif ss == GRB.StopSolLimit
        return MOI.SolutionLimit
    elseif ss == GRB.StopUser
        return MOI.Interrupted
    end
    return MOI.OtherError
end

function LQOI.lqs_primalstatus(model::GurobiSolverInstance)
    m = model.inner
    return MOI.FeasiblePoint
    # if GRB.is_mip(m)
    #     stat_mip = GRB.get_mip_status2(m)
    #     if stat_mip in [GRB.MIP_Solution, GRB.MIP_Optimal]
    #         return MOI.FeasiblePoint
    #     elseif GRB.MIP_Infeasible && GRB.hasdualray(m)
    #         return MOI.InfeasibilityCertificate
    #     elseif GRB.MIP_Unbounded && GRB.hasprimalray(m)
    #         return MOI.InfeasibilityCertificate
    #     elseif stat_mip in [GRB.MIP_LPOptimal, GRB.MIP_NoSolFound]
    #         return MOI.InfeasiblePoint
    #     end
    #     return MOI.UnknownResultStatus
    # else
    #     stat_lp = GRB.get_lp_status2(m)
    #     if stat_lp == GRB.LP_Optimal
    #         return MOI.FeasiblePoint
    #     elseif stat_lp == GRB.LP_Unbounded && GRB.hasprimalray(m)
    #         return MOI.InfeasibilityCertificate
    #     # elseif stat_lp == LP_Infeasible
    #     #     return MOI.InfeasiblePoint - Gurobi wont return
    #     # elseif cutoff//cutoffindual ???
    #     else
    #         return MOI.UnknownResultStatus
    #     end
    # end
end
function LQOI.lqs_dualstatus(model::GurobiSolverInstance)
    m = model.inner 
    if is_mip(model.inner)
        return MOI.UnknownResultStatus
    else
        return MOI.FeasiblePoint   
    end
    # if GRB.is_mip(m)
    #     return MOI.UnknownResultStatus
    # else
    #     stat_lp = GRB.get_lp_status2(m)
    #     if stat_lp == GRB.LP_Optimal
    #         return MOI.FeasiblePoint
    #     elseif stat_lp == GRB.LP_Infeasible && GRB.hasdualray(m)
    #         return MOI.InfeasibilityCertificate
    #     # elseif stat_lp == LP_Unbounded
    #     #     return MOI.InfeasiblePoint - Gurobi wont return
    #     # elseif cutoff//cutoffindual ???
    #     else
    #         return MOI.UnknownResultStatus
    #     end
    # end
end


# LQOI.lqs_getx!(m, place)
LQOI.lqs_getx!(m::GRB.Model, place) = GRB.get_dblattrarray!(place, m, "X", 1)

# LQOI.lqs_getax!(m, place)
function LQOI.lqs_getax!(m::GRB.Model, place)
    GRB.get_dblattrarray!(place, m, "Slack", 1)
    rhs = GRB.get_dblattrarray(m, "RHS", 1, num_constrs(m))
    for i in eachindex(place)
        place[i] = -place[i]+rhs[i]
    end
    nothing
end
# LQOI.lqs_getdj!(m, place)
LQOI.lqs_getdj!(m::GRB.Model, place) = GRB.get_dblattrarray!(place, m, "RC", 1)

# LQOI.lqs_getpi!(m, place)
LQOI.lqs_getpi!(m::GRB.Model, place) = GRB.get_dblattrarray!(place, m, "Pi", 1)

# LQOI.lqs_getobjval(m)
LQOI.lqs_getobjval(m::GRB.Model) = GRB.get_objval(m)

# LQOI.lqs_getbestobjval(m)
LQOI.lqs_getbestobjval(m::GRB.Model) = GRB.get_objval(m)

# LQOI.lqs_getmiprelgap(m)
function LQOI.lqs_getmiprelgap(m::GRB.Model)
    L = GRB.get_objval(m)
    U = GRB.get_objbound(m)
    return abs(U-L)/U
end

# LQOI.lqs_getitcnt(m)
LQOI.lqs_getitcnt(m::GRB.Model)  = GRB.get_iter_count(m)

# LQOI.lqs_getbaritcnt(m)
LQOI.lqs_getbaritcnt(m::GRB.Model) = GRB.get_barrier_iter_count(m)

# LQOI.lqs_getnodecnt(m)
LQOI.lqs_getnodecnt(m::GRB.Model) = GRB.get_node_count(m)

# LQOI.lqs_dualfarkas(m, place)
LQOI.lqs_dualfarkas!(m::GRB.Model, place) = GRB.getdualray!(m, place)

# LQOI.lqs_getray(m, place)
LQOI.lqs_getray!(m::GRB.Model, place) = GRB.getprimalray!(m, place)


MOI.free!(m::GurobiSolverInstance) = GRB.free_model(m.inner)

"""
    writeproblem(m::AbstractSolverInstance, filename::String)
Writes the current problem data to the given file.
Supported file types are solver-dependent.
"""
MOI.writeproblem(m::GurobiSolverInstance, filename::String, flags::String="") = GRB.write_model(m.inner, filename, flags)

end # module