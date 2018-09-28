within MbsLite.Quaternions;

function QToT "Converts quaternion to Rotation matrix."

  input  Real[4]     q;
  output Real[3, 3]  T;

algorithm

  T := [ q[1]^2 + q[2]^2 - q[3]^2 - q[4]^2,    2 * (q[2] * q[3] - q[1] * q[4]),      2 * (q[2] * q[4] + q[1] * q[3])
       ; 2 * (q[1] * q[4] + q[2] * q[3]),      q[1]^2 - q[2]^2 + q[3]^2 - q[4]^2,    2 * (q[3] * q[4] - q[1] * q[2])
       ; 2 * (q[2] * q[4] - q[1] * q[3]),      2 * (q[1] * q[2] + q[3] * q[4]),      q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2
       ] / (q * q);

  annotation(Inline=true);

end QToT;
