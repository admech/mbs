within MBS.Examples.OmniVehicle;

model RollerOnPlaneSlipping
  extends SlidingWithDryFriction;
  parameter Real R = 1;
  parameter Real R1 = R/sqrt(2);
  Real xi;
equation
  xi^2 = rhoB[2]^2 + rhoB[3]^2;
  (xi + R1)^2 + rhoB[1]^2 = R^2;
  rhoA[2] = 0;
  gradfA = {0, 1, 0};
  gradfB = 2*{rhoB[1], rhoB[2]*(xi + R1)/xi, rhoB[3]*(xi + R1)/xi};
end RollerOnPlaneSlipping;
