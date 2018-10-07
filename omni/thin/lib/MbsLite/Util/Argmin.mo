within MbsLite.Util;

function Argmin
  
  input  Real[:]    items;

  output Integer    argmin;

protected
  Integer n;
  Real    min;

algorithm
  n := size(items, 1);
  Assert
    ( n > 0
    , "Argmin received " + String(n) + " <= 0 items"
    , silent = true);

  argmin := 1;
  min    := items[argmin];

  for i in 2 : n loop
    if items[i] < min then
      min := items[i];
      argmin := i;
    end if;
  end for;

  print("argmin of " + StringA(items) + " is " + String(argmin));

end Argmin;

