function connectionFcn(src,~)
if src.Connected
   disp("Accepting the client connection request.")
else
   disp("Client has disconnected.")
end
end
