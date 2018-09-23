within MBS;

partial model BaseBody
  replaceable KinematicPort OutPort annotation (extent=[-10,-100; 10,-80],
      Placement(transformation(extent={{-20,-100},{20,-60}})));

  annotation (Icon, Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}}), graphics));
end BaseBody;
