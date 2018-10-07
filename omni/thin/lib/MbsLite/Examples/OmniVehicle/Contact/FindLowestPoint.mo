within MbsLite.Examples.OmniVehicle.Contact;

function FindLowestPoint "finds coords and velocity of the lowest point of the lowest roller of an omni-wheel in global coords"

  input  Real[3]    wheelCenter;
  input  Real       wheelRadius;

  input  Real[3]    lowestRollerCenter;
  input  Real[3]    lowestRollerVelocity;
  input  Real[3]    lowestRollerAngularVelocity;

  output Real[3]    coords;
  output Real[3]    velocity;

protected
  Real[3] lowestPoint;

algorithm
  
  // just take the point directly under wheel center
  lowestPoint := wheelCenter - wheelRadius * vertical;
  
  coords := lowestPoint;

  velocity := Euler
    ( lowestRollerCenter
    , lowestPoint
    , lowestRollerVelocity
    , lowestRollerAngularVelocity
    );

end FindLowestPoint;

