within MbsLite.Quaternions;

function QRot

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

end QRot;
