function [ LR ] = mrpprod( L, R)
[r ,c ] = size(L);

if (r==3 && c == 1) || (r==1 && c == 3)
    LR = ( (1 - dot(R(:),R(:))) * L(:) + (1 - dot(L(:),L(:))) * R(:) + 2* cross(L(:),R(:)) )/( 1 + dot(L(:),L(:))*dot(R(:),R(:)) - 2* dot(L(:),R(:))  );
elseif c == 3
    nc = r;
    LR = zeros(r,c);
    for iq = 1:nc
        Rsq = R(iq,:) * R(iq,:).';
        Lsq = L(iq,:) * L(iq,:).';
        LR(iq,:) = ( (1 - Rsq) * L(iq,:).' + (1 - Lsq) * R(iq,:).' + 2* cross(L(iq,:).',R(iq,:).') )/( 1 + Lsq*Rsq - 2* dot(L(iq,:).',R(iq,:).')  ).';
    end
elseif r == 3
    nc = c;
    LR = zeros(c,r);
    for iq = 1:nc
        Rsq = R(:,iq).' * R(:,iq);
        Lsq = L(:,iq).' * L(:,iq);
        LR(iq,:) = ( (1 - Rsq) * L(:,iq) + (1 - Lsq) * R(:,iq) + 2* cross(L(:,iq),R(:,iq)) )/( 1 + Lsq * Rsq - 2* dot(L(:,iq),R(:,iq))  );
    end
else
        error('Dimensions do not match');
end

end