within MBS.Examples.OmniVehicle;

model RollerOverHorizontalSurface
  extends Constraint;
  parameter Real alpha = 1;
  parameter Real R = 1;
  parameter Real R1 = R*cos(alpha);
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  parameter Real stiff = 100;
  parameter Real damp = 10;
  Real kappa;
  Real[3] vA;
  Real[3] vB;
  Real[3] vr;
  Real[3] vrtan;
  Real vrt;
  Real[3] RBt;
  Real RBn;
  SI.Position[3] rA;
  SI.Position[3] rB;
  Real h;
  Real[3] nA;
equation
  h = 0;//
  rA = InPortA.r + InPortA.T*rhoA;
  rB = InPortB.r + InPortB.T*rhoB;
  nA = {0, 1, 0};
  vA = InPortA.v + cross(InPortA.omega, rA - InPortA.r);
  vB = InPortB.v + cross(InPortB.omega, rB - InPortB.r);
  vr = vB - vA;
  vrn = vr*nA;
  vrtan = vr - vrn*nA;
  vrt = sqrt(vrtan*vrtan);
  RBn = OutPortB.F*nA;
  RBn = if h >= 0 then 0 else -stiff*h*sqrt(abs(h)) - damp*vrn;
  RBt = -fric*vrtan*(if vrt <= delta then 1/delta else 1/vrt)*RBn + kappa*nA;
  OutPortB.F = RBt + RBn*nA;
  OutPortA.P = rA;
  OutPortB.P = rB;
  OutPortB.M = zeros(3);
end RollerOverHorizontalSurface;
