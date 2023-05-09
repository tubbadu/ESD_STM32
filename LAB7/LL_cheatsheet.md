## STM32 LL cheat sheet

```c
LL_GPIO_ReadReg(__INSTANCE__,__REG__);
LL_GPIO_WriteReg(__INSTANCE__,__REG__,__VALUE__);
```

INSTANCE:

- `GPIOA`
- `GPIOB`
- `GPIOC`

REG:

- `GPIOx_MODER` -> I/O direction mode
  - `00` input
  - `01` output
  - `10` alternate function mode
  - `11` analog mode
- `GPIOx_TYPER` -> output type
  - `0` Output push-pull
  - `1` Open drain
- `GPIOx_PUPDR` -> push/pull type
  - `00` no push/pull
  - `01` pull up
  - `10` pull down
  - `11` nothing
- `GPIOx_IDR` -> contains the actual value read by that pin
- `GPIOx_ODR` -> contains the actual value to be written on that pin

> note: we will only program IDR and ODR, as the others are programmed by the IDE

```c
HAL_Delay(__time__);
```

