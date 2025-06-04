function flag = magOutlier(mag, L, tol)
    
    mag_mag = vecnorm(mag, 2, 1);

    if abs(mag_mag-L) > tol
        flag = 1;
    else
        flag = 0;
    end

end