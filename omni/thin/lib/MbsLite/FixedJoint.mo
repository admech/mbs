within MbsLite;

model FixedJoint
  extends Constraint;

  parameter String           name = "NOT INITIALIZED";

  parameter Real[3]          nA             "Axis in body A local";
  parameter Real[3]          nB             "Axis in body B local";
  parameter SI.Position[3]   rA             "Axis position in body A local";
  parameter SI.Position[3]   rB             "Axis position in body B local";

  parameter Boolean          hasFriction    = false;
  parameter Real             frictionCoeff  = inf;

  SI.Position[3]             RA             (each stateSelect = StateSelect.never) "Axis position in body A global";
  SI.Position[3]             RB             (each stateSelect = StateSelect.never) "Axis position in body B global";
  SI.Velocity[3]             vA             (each stateSelect = StateSelect.never) "Velocity of axis point in body A global";
  SI.Velocity[3]             vB             (each stateSelect = StateSelect.never) "Velocity of axis point in body B global";
  // Real[3]                    vr             "Relative velocity of axis points in bodies";
  // Real[3]                    ar             "Relative acceleration of axis points in bodies";

  SI.AngularAcceleration     lambda         (stateSelect = StateSelect.never) "Relative angular acceleration along the axis";
  SI.AngularVelocity         mu             (stateSelect = StateSelect.never) "Relative angular velocity along the axis";
  Real                       angle          (stateSelect = StateSelect.never) "Relative angle. Beware numerical error accumulation!";
  // SI.AngularVelocity[3]      omegar         "Relative angular velocity";
  SI.AngularAcceleration[3]  epsilonr       (each stateSelect = StateSelect.never) "Relative angular acceleration";

  Real[3]                    nAi            (each stateSelect = StateSelect.never) "Unit vector of joint axis w.r.t. the inertial frame, def by body A";
  Real[3]                    nBi            (each stateSelect = StateSelect.never) "Unit vector of joint axis w.r.t. the inertial frame, def by body B";

  SI.Torque                  M              (stateSelect = StateSelect.never) "Torque about joint axis";

initial equation
  AssertInitializedS(name, name, "name");

equation

  RA = InPortA.r + InPortA.T * rA;
  RB = InPortB.r + InPortB.T * rB;
  vA = Euler(InPortA.r, RA, InPortA.v, InPortA.omega);
  vB = Euler(InPortB.r, RB, InPortB.v, InPortB.omega);
  vA = vB;
  // vr = vB - vA;
  // der(vr) = ar;
  // ar = zeros(3);

  nAi = InPortA.T * nA;
  nBi = InPortB.T * nB;
/*
  for i in 1 : 3 loop
    assert
      ( CompareReal(nAi[i], nBi[i], absTol = 1e-2)
      , "looks like joint " + name + " is getting worse... axes in inertial coords should be same, but were:"
        +  " nAi = " + StringA(nAi)
        + ", nBi = " + StringA(nBi)
        + ", nBi - nAi = " + StringA({ (nBi[j] - nAi[j]) for j in 1 : 3 })
      );
  end for;
*/

  //  InPortA.epsilon = InPortB.epsilon;
  //  cross(InPortA.T * nA, InPortB.T*nB) = zeros(3);
  //  InPortB.T * nB = mu*(InPortA.T*nA);
  //  InPortB.omega - InPortA.omega = lambda * InPortA.T*nA;

  // fixme: why invent an omegar provided the cross in the next line just zeroes the collinear one ?
  // maybe it's need if one uncomments below equation with lambda...
  /*
  omegar = InPortB.omega - InPortA.omega;
  epsilonr = InPortB.epsilon - InPortA.epsilon - cross(InPortA.omega, omegar);
  */
  //  omegar = lambda * nAi; // is it here that omegar was needed ?
  epsilonr = InPortB.epsilon - InPortA.epsilon - cross(InPortA.omega, InPortB.omega);
  epsilonr = lambda * nAi;

  der(mu) = lambda;

  M = OutPortA.M * nAi;
  M = if hasFriction then -frictionCoeff * mu else 0;

  OutPortA.P = RA;
  OutPortB.P = RB;

  der(angle) = mu;

end FixedJoint;

