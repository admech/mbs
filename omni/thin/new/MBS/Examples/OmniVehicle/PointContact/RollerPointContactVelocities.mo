within MBS.Examples.OmniVehicle.PointContact;

partial model RollerPointContactVelocities
  extends RollerContactTracking;
  Real[3] vA;
  Real[3] vB;
  Real[3] relv;
  Real relvn;
  Real vAn;
  Real vBn;
  Real[3] vAt;
  Real[3] vBt;
  Real[3] relvt;
  Real relvtsqrt;
equation
  vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
  vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
  relv = vB - vA;
  relvn = relv*nA;
  vAn = vA*nA;
  vBn = vB*nA;
  vAt = vA - vAn*nA;
  vBt = vB - vBn*nA;
  relvt = vBt - vAt;
  relvtsqrt = sqrt(relvt*relvt);

  OutPortA.P = rA;
  OutPortB.P = rB;
end RollerPointContactVelocities;
