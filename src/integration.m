function i = integration(vector1,time,c)

arguments
    vector1 double
    time double
    c double
  end
  
  d = [0; cumsum(vector1.*diff(time))] + c;


end