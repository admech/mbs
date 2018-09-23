within MBS;

partial model ContactTracking
  extends Constraint;
  SI.Position[3] rA;
  SI.Position[3] rB;
  Real mu;
  Real lambda;
  Real[3] gradfA;
  Real[3] gradfB;
  Real[3] gradgA;
  Real[3] gradgB;
  Real[3] nA;
  Real[3] rhoA(start = {0, 0, 0});
  Real[3] rhoB(start = {0, 0, 0});
equation
  gradgB = lambda*gradgA;
  rB - rA = mu*gradgA;
  rA = InPortA.r + InPortA.T*rhoA;
  rB = InPortB.r + InPortB.T*rhoB;
  gradgA = InPortA.T*gradfA;
  gradgB = InPortB.T*gradfB;
  nA = gradgA/sqrt(gradgA*gradgA);
end ContactTracking;
