/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../J2735_201603.asn1"
 * 	`asn1c -S ../../skeletons -pdu=MessageFrame -fcompound-names -pdu=auto`
 */

#ifndef	_VerticalAccelerationThreshold_H_
#define	_VerticalAccelerationThreshold_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VerticalAccelerationThreshold {
	VerticalAccelerationThreshold_notEquipped	= 0,
	VerticalAccelerationThreshold_leftFront	= 1,
	VerticalAccelerationThreshold_leftRear	= 2,
	VerticalAccelerationThreshold_rightFront	= 3,
	VerticalAccelerationThreshold_rightRear	= 4
} e_VerticalAccelerationThreshold;

/* VerticalAccelerationThreshold */
typedef BIT_STRING_t	 VerticalAccelerationThreshold_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VerticalAccelerationThreshold_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VerticalAccelerationThreshold;
asn_struct_free_f VerticalAccelerationThreshold_free;
asn_struct_print_f VerticalAccelerationThreshold_print;
asn_constr_check_f VerticalAccelerationThreshold_constraint;
ber_type_decoder_f VerticalAccelerationThreshold_decode_ber;
der_type_encoder_f VerticalAccelerationThreshold_encode_der;
xer_type_decoder_f VerticalAccelerationThreshold_decode_xer;
xer_type_encoder_f VerticalAccelerationThreshold_encode_xer;
oer_type_decoder_f VerticalAccelerationThreshold_decode_oer;
oer_type_encoder_f VerticalAccelerationThreshold_encode_oer;
per_type_decoder_f VerticalAccelerationThreshold_decode_uper;
per_type_encoder_f VerticalAccelerationThreshold_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _VerticalAccelerationThreshold_H_ */
#include <asn_internal.h>
