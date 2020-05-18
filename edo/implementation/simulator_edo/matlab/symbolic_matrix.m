syms a1 a2 a3 c1 s1 c2 s2 c3 s3 c23 s23 pv1 pv2 pv3;

J = [-s1*(a2*c2 + a3*c23), -c1*(a2*s2 + a3*s23), -a3*c1*s23;
      c1*(a2*c2 + a3*c23), -s1*(a2*s2 + a3*s23), -a3*s1*s23;
      0,                   a2*c2 + a3*c23,       a3*c23];

J_inv = inv(J);

fprintf("\ndetJ:\n");
ccode(det(J))

pv = transpose([pv1, pv2, pv3]);


qv = J_inv*pv;

fprintf("\nqv[0]:\n");
ccode(qv(1))
fprintf("\nqv[1]:\n");
ccode(qv(2))
fprintf("\nqv[2]:\n");
ccode(qv(3))


