
import fs from 'fs';
import avrpizza from  'avr-pizza';
import Avrgirl from 'avrgirl-arduino';
import find from 'local-devices';


class ApiService {
    async hexCompiler(code:string) {
     console.log(find)
        fs.writeFileSync("temp.ino", code);
        // this.compile()
        var pack = {
            sketch: 'temp.ino',
          //  / libraries: ['D:/custom-ide/server/libs/'],
            board: 'uno'
          };
        avrpizza.compile(pack, function(error, hex) {
            if(error){console.log('error',error)}
            fs.writeFileSync('compiled.hex', hex);
            return hex;
          })

    }
    async burn(){
      const avrgirl = new Avrgirl({
        board: 'leonardo'
      });
      
      avrgirl.flash('compiled.hex', function (error) {
        if (error) {
          console.error(error);
        } else {
          console.info('done.');
        }
      });
      
    }
    // compile(){
    //     let sketch = {
    //         sketch: './temp.ino',
    //         board: 'uno',
    //         // builder: {
    //         //     location: 'C:\Program Files\Arduino IDE\Arduino IDE'
    //         //   }
    //       };
    //       avrpizza.compile(sketch, function(error, hex) {
    //         fs.writeFile('compiled.hex', hex, (err) => {
    //             if(err) throw err;
    //             console.log('File created');
    //         });
    //         // fs.appendFile('compiled.hex', hex, (err) => {
    //         //     if(err) throw err;
    //         //     console.log('Data has been added2!');
    //         // });
    //       });
    // }

};

const apiService = new ApiService();
module.exports = apiService;