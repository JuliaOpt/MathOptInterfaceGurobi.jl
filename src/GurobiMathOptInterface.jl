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



const SUPPORTED_OBJECTIVES = [
    LQOI.Linear,
    LQOI.Quad
]
const SUPPORTED_CONSTRAINTS = [
    (LQOI.Linear, LQOI.EQ),
    (LQOI.Linear, LQOI.LE),
    (LQOI.Linear, LQOI.GE),
    # (Linear, IV),
    (LQOI.Quad, LQOI.EQ),
    (LQOI.Quad, LQOI.LE),
    (LQOI.Quad, LQOI.GE),
    (LQOI.SinVar, LQOI.EQ),
    (LQOI.SinVar, LQOI.LE),
    (LQOI.SinVar, LQOI.GE),
    (LQOI.SinVar, LQOI.IV),
    (LQOI.SinVar, MOI.ZeroOne),
    (LQOI.SinVar, MOI.Integer),
    (LQOI.VecVar, MOI.SOS1),
    (LQOI.VecVar, MOI.SOS2),
    (LQOI.VecVar, MOI.Nonnegatives),
    (LQOI.VecVar, MOI.Nonpositives),
    (LQOI.VecVar, MOI.Zeros),
    (LQOI.VecLin, MOI.Nonnegatives),
    (LQOI.VecLin, MOI.Nonpositives),
    (LQOI.VecLin, MOI.Zeros)
]

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
        GRB.setparam!(m.inner, string(name), value)
    end
    # csi.inner.mipstart_effort = s.mipstart_effortlevel
    # if s.logfile != ""
    #     LQOI.lqs_setlogfile!(env, s.logfile)
    # end
    return m
end


lqs_supported_constraints(s::MOIGurobiSolver) = SUPPORTED_CONSTRAINTS
lqs_supported_objectives(s::MOIGurobiSolver) = SUPPORTED_OBJECTIVES
lqs_supported_constraints(s::GurobiSolverInstance) = SUPPORTED_CONSTRAINTS
lqs_supported_objectives(s::GurobiSolverInstance) = SUPPORTED_OBJECTIVES
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
LQOI.lqs_setparam!(m::GurobiSolverInstance, name, val) = GRB.setparam!(m.inner, string(name), val)

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
    for i in eachindex(V)
        if I[i] == J[i]
            V[i] /= 2
        end
    end
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
LQOI.lqs_lpopt!(m::GRB.Model) = (GRB.update_model!(m);GRB.optimize(m))

# LQOI.lqs_terminationstatus(m)
function LQOI.lqs_terminationstatus(model::GurobiSolverInstance)
    m = model.inner
    
    stat = get_status(m)

    if stat == :loaded
        return MOI.OtherError
    elseif stat == :optimal
        return MOI.Success
    elseif stat == :infeasible
        if hasdualray(m)
            return MOI.Success
        else
            return MOI.InfeasibleNoResult
        end
    elseif stat == :inf_or_unbd
        return MOI.InfeasibleOrUnbounded
    elseif stat == :unbounded
        if hasprimalray(m)
            return MOI.Success
        else
            return MOI.UnboundedNoResult
        end
    elseif stat == :cutoff
        return MOI.ObjectiveLimit
    elseif stat == :iteration_limit
        return MOI.IterationLimit
    elseif stat == :node_limit
        return MOI.NodeLimit
    elseif stat == :time_limit
        return MOI.TimeLimit
    elseif stat == :solution_limit
        return MOI.SolutionLimit
    elseif stat == :interrupted
        return MOI.Interrupted
    elseif stat == :numeric
        return MOI.NumericalError
    elseif stat == :suboptimal
        return MOI.OtherLimit
    elseif stat == :inprogress
        return MOI.OtherError
    elseif stat == :user_obj_limit
        return MOI.ObjectiveLimit
    end
    return MOI.OtherError
end


function LQOI.lqs_primalstatus(model::GurobiSolverInstance)
    m = model.inner

    stat = get_status(m)

    if stat == :optimal
        return MOI.FeasiblePoint
    elseif stat == :solution_limit
        return MOI.FeasiblePoint
    elseif stat in [:inf_or_unbd, :unbounded] && hasprimalray(m)
        return MOI.InfeasibilityCertificate
    elseif stat == :suboptimal
        return MOI.FeasiblePoint
    else 
        return MOI.UnknownResultStatus
    end
end
function LQOI.lqs_dualstatus(model::GurobiSolverInstance)
    m = model.inner 
    stat = get_status(m)
    
    if GRB.is_mip(m) || GRB.is_qcp(m)
        return MOI.UnknownResultStatus
    else
        if stat == :optimal
            return MOI.FeasiblePoint
        elseif stat == :solution_limit
            return MOI.FeasiblePoint
        elseif stat in [:inf_or_unbd, :infeasible] && hasdualray(m)
            return MOI.InfeasibilityCertificate
        elseif stat == :suboptimal
            return MOI.FeasiblePoint
        else 
            return MOI.UnknownResultStatus
        end  
    end
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
LQOI.lqs_dualfarkas!(m::GRB.Model, place) = GRB.get_dblattrarray!(place, m, "FarkasDual", 1)

function hasdualray(m::GRB.Model)
    try
        GRB.get_dblattrarray(m, "FarkasDual", 1, GRB.num_constrs(m))
        return true
    catch
        return false
    end
end

# LQOI.lqs_getray(m, place)
LQOI.lqs_getray!(m::GRB.Model, place) = GRB.get_dblattrarray!(place, m, "UnbdRay", 1)

function hasprimalray(m::GRB.Model)
    try
        GRB.get_dblattrarray(m, "UnbdRay", 1, GRB.num_vars(m))
        return true
    catch
        return false
    end
end

MOI.free!(m::GurobiSolverInstance) = GRB.free_model(m.inner)

"""
    writeproblem(m::AbstractSolverInstance, filename::String)
Writes the current problem data to the given file.
Supported file types are solver-dependent.
"""
MOI.writeproblem(m::GurobiSolverInstance, filename::String, flags::String="") = GRB.write_model(m.inner, filename, flags)


# blocked
MOI.addconstraint!(m::GurobiSolverInstance, func::LQOI.Linear, set::LQOI.IV) = error("not supported")
MOI.addconstraints!(m::GurobiSolverInstance, func::Vector{LQOI.Linear}, set::Vector{LQOI.IV}) = error("not supported")

MOI.cangetattribute(m::GurobiSolverInstance, any, c::LQOI.LCR{LQOI.IV}) = false
MOI.canmodifyconstraint(m::GurobiSolverInstance, c::LQOI.LCR{LQOI.IV}, chg) = false
MOI.candelete(m::GurobiSolverInstance, c::LQOI.LCR{LQOI.IV}) = false

end # module