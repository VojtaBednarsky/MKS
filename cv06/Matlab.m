close all
clear all
clc

data = 'ntc.csv';
TAB = csvread(data);

r = TAB(:,2)
t = TAB(:,1)

ad = 1024*((r)./(10+r))

plot(ad, t, 'o')
hold on

p = polyfit(ad, t, 10);

ad2 = 0:1023;
t2 = round(polyval(p, ad2), 1);
hold on, plot(ad2, t2, 'r');

dlmwrite('data.dlm', t2*10, ',');