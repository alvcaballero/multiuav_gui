export class utilsModel {
  static DateTime() {
    const dateObject = new Date();
    let stringdate = dateObject.toJSON(); //dateObject.toJSON().slice(0, -1).replace('T',' ');
    console.log('uav sincronize time' + stringdate);
    return { datetime: stringdate };
  }
}
