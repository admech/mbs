within MbsLite.Util.MassGeometry;

function CylinderOrthogonalMoi

  input Real m;
  input Real r;
  input Real l;

  output Real orthogonal;

algorithm

  orthogonal := m * 1/12 * (3 * (r^2) + l^2);

end CylinderOrthogonalMoi;
