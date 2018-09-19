within MBS;

partial model Rolling
  extends PointContact;
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  Real kappa;
equation
  vr = kappa*nA;
end Rolling;
