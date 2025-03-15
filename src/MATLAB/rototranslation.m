function new_points = rototranslation(points, t, theta)
    R = [cos(theta), -sin(theta); 
         sin(theta),  cos(theta)];

    new_points = (R * points' + t')';
end