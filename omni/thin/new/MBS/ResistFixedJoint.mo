within MBS;

model ResistFixedJoint
  extends Joint;
  parameter Stiffness c=1000 "Stiffness";
  parameter Viscosity d=1000 "Viscosity";
  SI.Angle phi(start = 0);
  SI.AngularVelocity dphi(start = 0);
equation
  der(phi) = dphi;
  der(dphi) = lambda;
  mu = 0;
  M = c*phi + d*dphi;
  annotation (Icon(
      Text(
        extent=[-52,26; 100,-24],
        style(color=3, rgbcolor={0,0,255}),
        string="ResistFixedJoint"),
      Line(points=[-80,-20; -70,-20], style(
          color=0,
          rgbcolor={0,0,0},
          gradient=2,
          fillColor=10,
          rgbfillColor={135,135,135})),
      Line(points=[-56,-20; -52,-20], style(
          color=0,
          rgbcolor={0,0,0},
          gradient=2,
          fillColor=10,
          rgbfillColor={135,135,135})),
      Rectangle(extent=[-76,-10; -56,-30], style(color=0, rgbcolor={0,0,0})),
      Rectangle(extent=[-66,-10; -56,-30], style(
          color=0,
          rgbcolor={0,0,0},
          fillColor=10,
          rgbfillColor={135,135,135})),
      Rectangle(extent=[-70,-10; -66,-30], style(
          color=0,
          rgbcolor={0,0,0},
          fillColor=0,
          rgbfillColor={0,0,0})),
      Line(points=[-80,20; -76,20; -72,10; -66,30; -60,10; -56,20; -52,20],
          style(
          color=0,
          rgbcolor={0,0,0},
          fillPattern=1))));

end ResistFixedJoint;
