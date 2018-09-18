within MBS;

partial model SlidingWithDryFriction
  extends PointContact;
  parameter Real delta = 10^(-6);
  parameter Real fric = 0.1;
  Real kappa;
equation
  RBt = -fric*vr*(if vrt <= delta then 1/delta else 1/vrt) + kappa*nA; // N is absent
  OutPortB.M = zeros(3);
end SlidingWithDryFriction;
