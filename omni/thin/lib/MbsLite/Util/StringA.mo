within MbsLite.Util;

function StringA

  input Real[:] array;
  input String delim = ", ";

  output String render;

protected
  String accumulator = "";
  Integer counter = 0;
algorithm

  for item in array loop
    accumulator := accumulator + String(item);
    counter := counter + 1;
    if counter < size(array, 1) then
      accumulator := accumulator + delim;
    end if;
  end for;

  render := "{ " + accumulator + " }";

end StringA;
