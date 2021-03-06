within MbsLite.Util;

function Euler
  annotation(Inline=true);

  input Real[3] origin;
  input Real[3] point;
  input Real[3] velocityOfOrigin;
  input Real[3] omega;

  output Real[3] velocityOfPoint;

algorithm

  velocityOfPoint := velocityOfOrigin + cross(omega, point - origin);

end Euler;
