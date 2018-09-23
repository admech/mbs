within MBS;

partial model PointContact
  extends ContactTracking;
  Real[3] vA;
  Real[3] vB;
  Real[3] vr;
  Real vrt;
  Real[3] RBt;
  Real RBn;
equation
  mu = 0;
  vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
  vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
  vr = vB - vA;
  vrt = sqrt(vr*vr);
  RBn = OutPortB.F*nA;
  OutPortB.F = RBt + RBn*nA;
  OutPortA.P = rA;
  OutPortB.P = rB;
end PointContact;
