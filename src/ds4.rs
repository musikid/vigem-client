use crate::*;
use std::borrow::Borrow;
use std::{marker, pin, thread};
use std::{fmt, mem, ptr};

mod button;
mod reports;

use winapi::shared::winerror;

pub use button::*;
pub use reports::*;

pub struct DSRequestNotification {
	client: Client,
	ds4rn: bus::RequestNotification,
	_unpin: marker::PhantomPinned,
}


impl DSRequestNotification {
	/// Returns if the underlying target is still attached.
	#[inline]
	pub fn is_attached(&self) -> bool {
		match self.ds4rn.buffer {
			bus::RequestNotificationVariant::DS4(ref buffer) => buffer.SerialNo != 0,
			#[allow(unreachable_patterns)]
			_ => unreachable!()
		}
	}

	/// Spawns a thread to handle the notifications.
	///
	/// The callback `f` is invoked for every notification.
	///
	/// Returns a [`JoinHandle`](thread::JoinHandle) for the created thread.
	/// It is recommended to join the thread after the target from which the notifications are requested is dropped.
	#[inline]
	pub fn spawn_thread<F: FnMut(&DSRequestNotification, bus::DS4OutputReport) + Send + 'static>(self, mut f: F) -> thread::JoinHandle<()> {
		thread::spawn(move || {
			// Safety: the request notification object is not accessible after it is pinned
			let mut reqn = self;
			let mut reqn = unsafe { pin::Pin::new_unchecked(&mut reqn) };
			loop {
				reqn.as_mut().request();
				let result = reqn.as_mut().poll(true);
				match result {
					Ok(None) => {},
					Ok(Some(data)) => f(&reqn, data),
					// When the target is dropped the notification request is aborted
					Err(_) => break,
				}
			}
		})
	}

	/// Requests a notification.
	#[inline(never)]
	pub fn request(self: pin::Pin<&mut Self>) {
		unsafe {
			let device = self.client.device;
			let ds4rn = &mut self.get_unchecked_mut().ds4rn;
			match ds4rn.buffer {
				bus::RequestNotificationVariant::DS4(ref mut buffer) => {
					if buffer.SerialNo != 0 {
						ds4rn.ioctl(device);
					}
				},
				#[allow(unreachable_patterns)]
				_ => unreachable!()
			}
		}
	}

	/// Polls the request for notifications.
	///
	/// If `wait` is true this method will block until a notification is received.
	/// Else returns immediately if no notification is received yet.
	///
	/// Returns:
	///
	/// * `Ok(None)`: When `wait` is false and there is no notification yet.
	/// * `Ok(Some(_))`: The notification was successfully received.  
	///   Another request should be made or any other calls to `poll` return the same result.
	/// * `Err(OperationAborted)`: The underlying target was unplugged causing any pending notification requests to abort.
	/// * `Err(_)`: An unexpected error occurred.
	#[inline(never)]
	pub fn poll(self: pin::Pin<&mut Self>, wait: bool) -> Result<Option<bus::DS4OutputReport>, Error> {
		unsafe {
			let device = self.client.device;
			let ds4rn = &mut self.get_unchecked_mut().ds4rn;
			match ds4rn.poll(device, wait) {
				Ok(()) => {
					match &ds4rn.buffer {
						bus::RequestNotificationVariant::DS4(buffer) => {
							Ok(Some(bus::DS4OutputReport {
								small_motor: buffer.Report.small_motor,
								large_motor: buffer.Report.large_motor,
								lightbar_color: buffer.Report.lightbar_color,
							}))
						},
						#[allow(unreachable_patterns)]
						_ => unreachable!()
					}
				},
				Err(winerror::ERROR_IO_INCOMPLETE) => Ok(None),
				Err(winerror::ERROR_OPERATION_ABORTED) => {
					// Operation was aborted, fail all future calls
					// The is aborted when the underlying target is unplugged
					// This has the potential for a race condition:
					//  What happens if a new target is plugged inbetween calls to poll and request...
					match ds4rn.buffer {
						bus::RequestNotificationVariant::DS4(ref mut buffer) => { buffer.SerialNo = 0; },
						#[allow(unreachable_patterns)]
						_ => unreachable!()
					}
					Err(Error::OperationAborted)
				},
				Err(err) => Err(Error::WinError(err)),
			}
		}
	}
}
unsafe impl Sync for DSRequestNotification {}
unsafe impl Send for DSRequestNotification {}

impl fmt::Debug for DSRequestNotification {
	fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
		let buffer = match &self.ds4rn.buffer {
			bus::RequestNotificationVariant::DS4(buffer) => buffer,
			#[allow(unreachable_patterns)]
			_ => unreachable!()
		};
		f.debug_struct("DSRequestNotification")
			.field("client", &format_args!("{:?}", self.client))
			.field("serial_no", &buffer.SerialNo)
			.finish()
	}
}


impl Drop for DSRequestNotification {
	fn drop(&mut self) {
		unsafe {
			let this = pin::Pin::new_unchecked(self);
			let serial_no = match &this.ds4rn.buffer {
				bus::RequestNotificationVariant::DS4(buffer) => buffer.SerialNo,
				#[allow(unreachable_patterns)]
				_ => unreachable!()
			};
			if serial_no != 0 {
				let device = this.client.device;
				let ds4rn = &mut this.get_unchecked_mut().ds4rn;
				let _ = ds4rn.cancel(device);
			}
		}
	}
}


/// A virtual Sony DualShock 4 (wired).
pub struct DualShock4Wired<CL: Borrow<Client>> {
	client: CL,
	event: Event,
	serial_no: u32,
	id: TargetId,
}

impl<CL: Borrow<Client>> DualShock4Wired<CL> {
	/// Creates a new instance.
	#[inline]
	pub fn new(client: CL, id: TargetId) -> DualShock4Wired<CL> {
		let event = Event::new(false, false);
		DualShock4Wired {
			client,
			event,
			serial_no: 0,
			id,
		}
	}

	/// Returns if the controller is plugged in.
	#[inline]
	pub fn is_attached(&self) -> bool {
		self.serial_no != 0
	}

	/// Returns the id the controller was constructed with.
	#[inline]
	pub fn id(&self) -> TargetId {
		self.id
	}

	/// Returns the client.
	#[inline]
	pub fn client(&self) -> &CL {
		&self.client
	}

	/// Unplugs and destroys the controller, returning the client.
	#[inline]
	pub fn drop(mut self) -> CL {
		let _ = self.unplug();

		unsafe {
			let client = (&self.client as *const CL).read();
			ptr::drop_in_place(&mut self.event);
			mem::forget(self);
			client
		}
	}

	/// Plugs the controller in.
	#[inline(never)]
	pub fn plugin(&mut self) -> Result<(), Error> {
		if self.is_attached() {
			return Err(Error::AlreadyConnected);
		}

		self.serial_no = unsafe {
			let mut plugin = bus::PluginTarget::ds4_wired(1, self.id.vendor, self.id.product);
			let device = self.client.borrow().device;

			// Yes this is how the driver is implemented
			while plugin.ioctl(device, self.event.handle).is_err() {
				plugin.SerialNo += 1;
				if plugin.SerialNo >= u16::MAX as u32 {
					return Err(Error::NoFreeSlot);
				}
			}

			plugin.SerialNo
		};

		Ok(())
	}

	/// Unplugs the controller.
	#[inline(never)]
	pub fn unplug(&mut self) -> Result<(), Error> {
		if !self.is_attached() {
			return Err(Error::NotPluggedIn);
		}

		unsafe {
			let mut unplug = bus::UnplugTarget::new(self.serial_no);
			let device = self.client.borrow().device;
			unplug.ioctl(device, self.event.handle)?;
		}

		self.serial_no = 0;
		Ok(())
	}

	/// Waits until the virtual controller is ready.
	///
	/// Any updates submitted before the virtual controller is ready may return an error.
	#[inline(never)]
	pub fn wait_ready(&mut self) -> Result<(), Error> {
		if !self.is_attached() {
			return Err(Error::NotPluggedIn);
		}

		unsafe {
			let mut wait = bus::WaitDeviceReady::new(self.serial_no);
			let device = self.client.borrow().device;
			wait.ioctl(device, self.event.handle)?;
		}

		Ok(())
	}

	/// Updates the virtual controller state.
	#[inline(never)]
	pub fn update(&mut self, report: &DS4Report) -> Result<(), Error> {
		if !self.is_attached() {
			return Err(Error::NotPluggedIn);
		}

		unsafe {
			let mut dsr = bus::DS4SubmitReport::new(self.serial_no, *report);
			let device = self.client.borrow().device;
			dsr.ioctl(device, self.event.handle)?;
		}

		Ok(())
	}

	/// Updates the virtual controller state using the extended report.
	#[inline(never)]
	pub fn update_ex(&mut self, report: &DS4ReportEx) -> Result<(), Error> {
		if !self.is_attached() {
			return Err(Error::NotPluggedIn);
		}

		unsafe {
			let mut dsr = bus::DS4SubmitReportEx::new(self.serial_no, *report);
			let device = self.client.borrow().device;
			dsr.ioctl(device, self.event.handle)?;
		}

		Ok(())
	}

	/// Request notification.
	///
	/// See examples/notification.rs for a complete example how to use this interface.
	///
	/// Do not create more than one request notification per target.
	/// Notifications may get lost or received by one or more listeners.
	#[inline(never)]
	pub fn request_notification(&mut self) -> Result<DSRequestNotification, Error> {
		if !self.is_attached() {
			return Err(Error::NotPluggedIn);
		}

		let client = self.client.borrow().try_clone()?;
		let ds4rn = bus::RequestNotification::new(bus::RequestNotificationVariant::DS4(bus::DS4RequestNotification::new(self.serial_no)));

		Ok(DSRequestNotification { client, ds4rn, _unpin: marker::PhantomPinned })
	}
}

impl<CL: Borrow<Client>> fmt::Debug for DualShock4Wired<CL> {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		f.debug_struct("DualShock4Wired")
			.field("serial_no", &self.serial_no)
			.field("vendor_id", &self.id.vendor)
			.field("product_id", &self.id.product)
			.finish()
	}
}

impl<CL: Borrow<Client>> Drop for DualShock4Wired<CL> {
	#[inline]
	fn drop(&mut self) {
		let _ = self.unplug();
	}
}
