within MbsLite.Quaternions;

function QMult

  input Real q1[4];
  input Real q2[4];
  output Real q3[4];

protected
  Real a1;
  Real a2;
  Real A1[3];
  Real A2[3];
  Real A3[3];

algorithm

  a1 := q1[1];
  a2 := q2[1];

  A1 := { q1[2], q1[3], q1[4] };
  A2 := { q2[2], q2[3], q2[4] };

  A3 := a1 * A2 + a2 * A1 + cross(A1, A2);

  q3 := { a1 * a2 - A1 * A2, A3[1], A3[2], A3[3] };

end QMult;
