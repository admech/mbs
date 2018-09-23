within MBS;

model RigidBodyEnergized
  extends RigidBody;
  SI.Energy Kt;
  SI.Energy Kr;
  SI.Energy K;
equation
  Kt = 0.5*m*v*v;
  Kr = 0.5*I*omega*omega;
  K = Kt + Kr;
end RigidBodyEnergized;
