% vehicle 2dof dynamic error model
function [kr,err] = error_calc(x,y,phi,vx,vy,phi_dot,xr,yr,thetar,kappar)
    n=length(xr);
    d_min=(x-xr(1))^2+(y-yr(1))^2;
    min=1;
    for i=1:n
        d=(x-xr(i))^2+(y-yr(i))^2;
        if d<d_min
            d_min=d;
            min=i;
        end
    end
    dmin=min;
    tor=[cos(thetar(dmin));sin(thetar(dmin))];
    nor=[-sin(thetar(dmin));cos(thetar(dmin))];
    d_err=[x-xr(dmin);y-yr(dmin)];
    ed=nor'*d_err;
    es=tor'*d_err;
    %projection_point_thetar=thetar(dmin);%apollo
    projection_point_thetar=thetar(dmin)+kappar(dmin)*es;
    ed_dot=vy*cos(phi-projection_point_thetar)+vx*sin(phi-projection_point_thetar);
    %%%%%%%%%
    ephi=sin(phi-projection_point_thetar);
    %%%%%%%%%
    s_dot=vx*cos(phi-projection_point_thetar)-vy*sin(phi-projection_point_thetar);
    s_dot=s_dot/(1-kappar(dmin)*ed);
    ephi_dot=phi_dot-kappar(dmin)*s_dot;
    kr=kappar(dmin);
    err=[ed;ed_dot;ephi;ephi_dot];

end
