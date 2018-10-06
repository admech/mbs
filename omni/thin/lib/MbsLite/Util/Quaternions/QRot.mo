within MbsLite.Util.Quaternions;

function QRot "quaternion of rotation by angle about axis CW+ looking along the axis"

  input  Real    angle;
  input  Real[3] axis;
  output Real[4] q;

protected
  Real s;

algorithm
  
  s := sin(angle / 2);
  q :=
    { cos(angle / 2)
    , s * axis[1]
    , s * axis[2]
    , s * axis[3]
    };

  annotation(Inline=true);

end QRot;
