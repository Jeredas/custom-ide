# custom-ide  
To run project install npm packages for cleint and server.
To start use npm run client & npm run server.  
# How in works  
React-Ace provides <AceEditor> component where you could include different options: languages, themes, autocomplete, syntaxcheck and etc.  
When your code is ready for compiling, press save sketch. This will trigger endpoint and server will save file with .ino extension. Then avr-pizza will compile .ino file to .hex according to type of arduino card.
Now everything is hardcoded wich means it creates sketches only in C++ and for Arduino UNO card.  


# Libraries  
---  
Client side
"ace-builds": "^1.4.14",  

"ace-code-editor": "^1.2.3",  

"react-ace": "^9.5.0",  

https://ace.c9.io/#nav=about  
  
---  
Serverside  
"avr-pizza": "^0.3.5",  
https://github.com/noopkat/avr-pizza