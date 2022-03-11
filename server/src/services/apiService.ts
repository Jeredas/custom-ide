
import fs from 'fs';
import avrpizza from  'avr-pizza';



class ApiService {
    hexCompiler(code:string) {
        fs.open('./temp/temp.ino', 'w', (err) => {
            if(err) throw err;
            console.log('File created');
        });
        fs.appendFile('.temp/temp.ino', code, (err) => {
            if(err) throw err;
            console.log('Data has been added1!');
        });
        this.compile()
        
    }
    compile(){
        let sketch = {
            sketch: 'temp.ino',
            board: 'uno'
          };
          avrpizza.compile(sketch, function(error, hex) {
              console.log(hex)
            fs.open('compiled.hex', 'w', (err) => {
                if(err) throw err;
                console.log('File created');
            });
            fs.appendFile('compiled1.hex', hex, (err) => {
                if(err) throw err;
                console.log('Data has been added2!');
            });
          });
    }

};

const apiService = new ApiService();
module.exports = apiService;