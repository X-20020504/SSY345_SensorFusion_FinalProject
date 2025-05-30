function flag = accOutlier(acc, tol)
    
    acc_mag = vecnorm(acc, 2, 1);

    if (acc_mag-9.81) > tol
        flag = 1;
    else
        flag = 0;
    end

end