within MBS;

function QMult1 "No protected - for array construction"
  input Real q1[4];
  input Real q2[4];
  output Real q3[4];
algorithm
  q3 := {q1[1]*q2[1] - {q1[2],q1[3],q1[4]}*{q2[2],q2[3],q2[4]},
         (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]},{q2[2],q2[3],q2[4]}))[1],
         (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]}, {q2[2],q2[3],q2[4]}))[2],
         (q1[1]*{q2[2],q2[3],q2[4]} + q2[1]*{q1[2],q1[3],q1[4]} + cross({q1[2],q1[3],q1[4]}, {q2[2],q2[3],q2[4]}))[3]};
  annotation(Inline=true);
end QMult1;
