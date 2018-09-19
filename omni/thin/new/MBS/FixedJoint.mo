within MBS;

model FixedJoint
  extends Joint;
equation
  mu = 0;
  M = 0;
  // Ideal joint
  annotation (Icon(
      Text(
        extent=[-50,36; 96,-38],
        style(color=3, rgbcolor={0,0,255}),
        string="%name")));
end FixedJoint;
