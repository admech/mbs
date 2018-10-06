within MbsLite.Util.MassGeometry;

function CylinderAxialMoi

  input Real m;
  input Real r;

  output Real axial;

algorithm

  axial      := m * (r^2) / 2;

end CylinderAxialMoi;
