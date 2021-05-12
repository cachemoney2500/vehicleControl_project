function y = lin_map(ymin,ymax,xmin,xmax,x)
    m = (ymax-ymin)/(xmax-xmin);
    b = ymax-m*xmax;
    y = m*x+b;
end