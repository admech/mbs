within MBS;

model PrismaticServoJoint
  extends Joint;
  input SI.Acceleration Control;
  SI.Position shift(start = 0);
  SI.Velocity dshift(start = 0);
equation
  der(shift) = dshift;
  der(dshift) = mu;
  lambda = 0;
  mu = Control;
  annotation (Icon(
      Text(
        extent=[-48,36; 98,-38],
        style(color=3, rgbcolor={0,0,255}),
        string="%name")));
end PrismaticServoJoint;
