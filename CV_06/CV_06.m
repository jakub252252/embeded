clear all;
close all;
clc;

n=csvread("ntc.csv");
r=n(:,1);
t=n(:,2);
ad=(r./(r+10)).*2^10;
p = polyfit(ad, t, 10);

ad2 = 0:1023;
t2 = round(polyval(p, ad2), 1);

figure;
plot(ad, t, 'bo');
hold on;
plot(ad2, t2, 'r');