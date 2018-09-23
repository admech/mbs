within MbsLite.Examples.OmniVehicle.PointContact;

model FixedJoint
  extends Constraint;

  parameter Real[3]        nA;
  parameter Real[3]        nB;
  parameter SI.Position[3] rA = zeros(3)    "Constraint position in body A";
  parameter SI.Position[3] rB = zeros(3)    "Constraint position in body B";

  SI.Position[3]             RA;
  SI.Position[3]             RB;
  SI.Velocity[3]             vA;
  SI.Velocity[3]             vB;
  SI.AngularAcceleration     lambda;
  // SI.AngularVelocity         mu;
  SI.AngularVelocity[3]      omegar         "Relative angular velocity";
  SI.AngularAcceleration[3]  epsilonr       "Relative angular acceleration";

  Real nAi[3] "Unit vector of joint axis w. r. t. inertial frame";

  SI.Torque M "Torque about joint axis";

equation

  RA = InPortA.r + InPortA.T * rA;
  RB = InPortB.r + InPortB.T * rB;
  vA = InPortA.v + cross(InPortA.omega, InPortA.T * rA);
  vB = InPortB.v + cross(InPortB.omega, InPortB.T * rB);
  vA = vB;

  nAi = InPortA.T * nA;

  //  InPortA.epsilon = InPortB.epsilon;
  //  cross(InPortA.T * nA, InPortB.T*nB) = zeros(3);
  //  InPortB.T * nB = mu*(InPortA.T*nA);
  //  InPortB.omega - InPortA.omega = lambda * InPortA.T*nA;

  omegar = InPortB.omega - InPortA.omega;
  //  omegar = lambda * nAi;
  epsilonr = InPortB.epsilon - InPortA.epsilon - cross(InPortA.omega, omegar);
  epsilonr = lambda * nAi;

  M = OutPortA.M * nAi;
  M = 0;

  OutPortA.P = RA;
  OutPortB.P = RB;

end FixedJoint;
