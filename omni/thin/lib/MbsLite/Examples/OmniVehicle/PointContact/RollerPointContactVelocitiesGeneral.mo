within MbsLite.Examples.OmniVehicle.PointContact;

partial model RollerPointContactVelocitiesGeneral
  extends RollerContactTrackingGeneral;

  parameter String   name = "NOT INITIALIZED";

  Real[3]  vA;
  Real[3]  vB;
  Real[3]  relv;
  Real     relvn;
  Real     vAn;
  Real     vBn;
  Real[3]  vAt;
  Real[3]  vBt;
  Real[3]  relvt;
  Real     relvtsqrt;

equation

  vA          = Euler(InPortA.r, rA, InPortA.v, InPortA.omega);
  vB          = Euler(InPortB.r, rB, InPortB.v, InPortB.omega);
  relv        = vB - vA;

  vAn         = vA * nA;
  vBn         = vB * nA;
  relvn       = relv * nA;

  vAt         = vA - vAn * nA;
  vBt         = vB - vBn * nA;
  relvt       = vBt - vAt;
  relvtsqrt   = norm(relvt);

  OutPortA.P  = rA;
  OutPortB.P  = rB;

end RollerPointContactVelocitiesGeneral;

