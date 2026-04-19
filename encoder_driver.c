/*
 * encoder_driver.c – Optical encoder driver for ELEC 424 Final Project
 *
 * Modified from the ELEC 424 Project 2 button driver.
 * Uses the kernel gpiod API (required by assignment) instead of the
 * deprecated gpio_request/gpio_to_irq approach.
 *
 * What this driver does:
 *   - Binds to the "elec424,optical-encoder" device tree node
 *   - Requests the GPIO declared in that node and maps it to an IRQ
 *   - On each rising edge (encoder hole passes sensor), records the
 *     elapsed time since the previous pulse
 *   - Rejects intervals < MIN_PULSE_NS to debounce mechanical noise
 *   - Calculates wheel RPM and exports it as a module parameter
 *     readable from user-space via:
 *       /sys/module/encoder_driver/parameters/speed_rpm
 *
 * Build: run  make  in this directory (see Makefile)
 * Load:  sudo insmod encoder_driver.ko
 * Read:  cat /sys/module/encoder_driver/parameters/speed_rpm
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>   /* gpiod_get, gpiod_to_irq */
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/atomic.h>
#include <linux/of.h>              /* of_device_id, MODULE_DEVICE_TABLE */

#define DRIVER_NAME      "optical-encoder"

/* Number of transparent slots in the encoder wheel.
 * Adjust to match the physical encoder provided in PCF3. */
#define ENCODER_HOLES    20

/* Minimum valid inter-pulse interval (5 ms).
 * Pulses arriving faster than this are treated as switch bounce / EMI noise. */
#define MIN_PULSE_NS     (5ULL * 1000000ULL)

/* If no pulse is seen for this long, report speed = 0 (car stopped). */
#define STALE_NS         (500ULL * 1000000ULL)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ELEC424 Team");
MODULE_DESCRIPTION("Optical encoder driver – measures RC car wheel speed");
MODULE_VERSION("1.0");

/* Module parameters – readable from sysfs by the Python lane-keeping script */
static int speed_rpm         = 0;
static int pulse_interval_ms = 0;
module_param(speed_rpm,         int, S_IRUGO);
module_param(pulse_interval_ms, int, S_IRUGO);
MODULE_PARM_DESC(speed_rpm,         "Current wheel speed in RPM");
MODULE_PARM_DESC(pulse_interval_ms, "Time between encoder pulses in ms");

/* GPIO descriptor obtained from the device tree node */
static struct gpio_desc *encoder_gpiod;

/* Timestamp of the most recent valid pulse (updated in IRQ context) */
static ktime_t last_pulse_ktime;

/* ── Interrupt Handler ──────────────────────────────────────────────────── */

static irqreturn_t encoder_irq_handler(int irq, void *dev_id)
{
    ktime_t now      = ktime_get();
    u64     interval_ns;

    /* Compute elapsed nanoseconds since last pulse */
    interval_ns = (u64)ktime_to_ns(ktime_sub(now, last_pulse_ktime));

    /* Reject glitches / switch-bounce below debounce threshold */
    if (interval_ns < MIN_PULSE_NS)
        return IRQ_HANDLED;

    /* Update module parameters visible to user-space */
    pulse_interval_ms = (int)(interval_ns / 1000000ULL);

    /* RPM = 60 s/min * 1e9 ns/s / (interval_ns * holes_per_rev) */
    speed_rpm = (int)(60ULL * 1000000000ULL / (interval_ns * (u64)ENCODER_HOLES));

    last_pulse_ktime = now;
    return IRQ_HANDLED;
}

/* ── Platform Driver Probe / Remove ────────────────────────────────────── */

static int encoder_probe(struct platform_device *pdev)
{
    int irq, ret;

    /* Retrieve the GPIO descriptor from the device tree "gpios" property */
    encoder_gpiod = devm_gpiod_get(&pdev->dev, NULL, GPIOD_IN);
    if (IS_ERR(encoder_gpiod)) {
        dev_err(&pdev->dev, "Failed to acquire encoder GPIO: %ld\n",
                PTR_ERR(encoder_gpiod));
        return PTR_ERR(encoder_gpiod);
    }

    /* Translate GPIO to an IRQ number */
    irq = gpiod_to_irq(encoder_gpiod);
    if (irq < 0) {
        dev_err(&pdev->dev, "gpiod_to_irq failed: %d\n", irq);
        return irq;
    }

    /* Request the IRQ; fire on rising edge (encoder hole unblocks IR beam) */
    ret = devm_request_irq(&pdev->dev, irq, encoder_irq_handler,
                           IRQF_TRIGGER_RISING, DRIVER_NAME, NULL);
    if (ret) {
        dev_err(&pdev->dev, "devm_request_irq failed for IRQ %d: %d\n", irq, ret);
        return ret;
    }

    /* Initialise timestamp so the first measured interval is plausible */
    last_pulse_ktime = ktime_get();

    dev_info(&pdev->dev, "Optical encoder driver loaded; IRQ %d\n", irq);
    return 0;
}

static void encoder_remove(struct platform_device *pdev)
{
    /* devm resources (GPIO + IRQ) are freed automatically by devres */
    speed_rpm         = 0;
    pulse_interval_ms = 0;
    dev_info(&pdev->dev, "Optical encoder driver removed\n");
}

/* ── Device-Tree Match Table ────────────────────────────────────────────── */

static const struct of_device_id encoder_of_match[] = {
    { .compatible = "elec424,optical-encoder" },
    { }   /* sentinel */
};
MODULE_DEVICE_TABLE(of, encoder_of_match);

/* ── Platform Driver Registration ───────────────────────────────────────── */

static struct platform_driver encoder_platform_driver = {
    .probe  = encoder_probe,
    .remove = encoder_remove,
    .driver = {
        .name           = DRIVER_NAME,
        .of_match_table = encoder_of_match,
    },
};

/* module_platform_driver() expands to module_init / module_exit boilerplate */
module_platform_driver(encoder_platform_driver);
