use embedded_graphics::{
    mono_font::{ascii::FONT_6X9, MonoTextStyle},
    pixelcolor::{PixelColor},
    prelude::*,
    primitives::{Rectangle, PrimitiveStyle},
    text::Text,
    Drawable,
};



struct Button<'a, C: PixelColor> {
    top_left: Point,
    size: Size,
    bg_color: C,
    fg_color: C,
    text: &'a str,
}

impl<C> Drawable for Button<'_, C>
where
    C: PixelColor,
{
    type Color = C;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = C>,
    {
        Rectangle::new(self.top_left, self.size)
            .into_styled(PrimitiveStyle::with_fill(self.bg_color))
            .draw(target)?;

        let style = MonoTextStyle::new(&FONT_6X9, self.fg_color);

        Text::new(self.text, Point::new(6, 13), style).draw(target)?;

        Ok(())
    }
}


