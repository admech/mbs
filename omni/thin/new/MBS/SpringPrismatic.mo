within MBS;

model SpringPrismatic
  extends Joint;
  parameter Stiffness c = 1000 "Stiffness";
  parameter Viscosity d = 1000 "Viscosity";
  SI.Position nu(start = 0);
  SI.Velocity dnu(start = 0);
equation
  der(nu) = dnu;
  der(dnu) = mu;
  F = c*nu + d*dnu;
  lambda = 0;
  annotation (Icon(
      Text(
        extent=[-52,34; 100,-32],
        style(color=3, rgbcolor={0,0,255}),
        string="SpringPrismatic"),
      Line(points=[-80,20; -76,20; -72,10; -66,30; -60,10; -56,20; -52,20],
          style(
          color=0,
          rgbcolor={0,0,0},
          fillPattern=1)),
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
          rgbfillColor={0,0,0}))));

end SpringPrismatic;
