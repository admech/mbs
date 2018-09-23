within MBS;

model FreeJoint
  extends Joint;
equation
  F = 0;
  M = 0;
  // Ideal joint
  annotation (Icon(Text(
        extent=[-50,36; 98,-38],
        style(color=3, rgbcolor={0,0,255}),
        string="FreeJoint")));
end FreeJoint;
