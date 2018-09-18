within MBS;

model FixedServoJoint
  extends Joint;
  input SI.AngularAcceleration Control;
  // Control
  SI.Angle phi(start=0);
  SI.AngularVelocity dphi(start=0);
initial equation
  //  omegar = dphi*nAi;
equation
  der(phi) = dphi;
  //  dphi = omegar*nAi;
  der(dphi) = lambda;
  //  der(phi) = lambda;
  lambda = Control;
  mu = 0;
  // Fixed joint
  annotation (Icon(Text(
        extent=[-52,34; 100,-32],
        style(color=3, rgbcolor={0,0,255}),
        string="FixedServoJoint")));
end FixedServoJoint;
