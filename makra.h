#ifndef MAKRA_H_
#define MAKRA_H_
// Makra upraszczaj�ce dost�p do port�w
// *** PORT
#define PORT(x) SPORT(x)
#define SPORT(x) (PORT##x)
// *** PIN
#define PIN(x) SPIN(x)
#define SPIN(x) (PIN##x)
// *** DDR
#define DDR(x) SDDR(x)
#define SDDR(x) (DDR##x)
#endif /* MAKRA_H_ */
