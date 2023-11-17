export class utilsModel {
  static DateTime() {
    const dateObject = new Date();

    // Get the date components
    const year = dateObject.getFullYear();
    const month = ('0' + (dateObject.getMonth() + 1)).slice(-2); // Months are zero-based
    const day = ('0' + dateObject.getDate()).slice(-2);

    // Get the time components
    const hours = ('0' + dateObject.getHours()).slice(-2);
    const minutes = ('0' + dateObject.getMinutes()).slice(-2);

    // Form the desired string
    const formattedString = `${year}-${month}-${day} ${hours}:${minutes}`;
    return { datetime: formattedString };
  }
}
