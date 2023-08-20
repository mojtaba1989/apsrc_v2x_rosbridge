/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../J2735_201603.asn1"
 * 	`asn1c -S ../../skeletons -pdu=MessageFrame -fcompound-names -pdu=auto`
 */

#ifndef	_PersonalDeviceUserType_H_
#define	_PersonalDeviceUserType_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PersonalDeviceUserType {
	PersonalDeviceUserType_unavailable	= 0,
	PersonalDeviceUserType_aPEDESTRIAN	= 1,
	PersonalDeviceUserType_aPEDALCYCLIST	= 2,
	PersonalDeviceUserType_aPUBLICSAFETYWORKER	= 3,
	PersonalDeviceUserType_anANIMAL	= 4
	/*
	 * Enumeration is extensible
	 */
} e_PersonalDeviceUserType;

/* PersonalDeviceUserType */
typedef long	 PersonalDeviceUserType_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PersonalDeviceUserType_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PersonalDeviceUserType;
extern const asn_INTEGER_specifics_t asn_SPC_PersonalDeviceUserType_specs_1;
asn_struct_free_f PersonalDeviceUserType_free;
asn_struct_print_f PersonalDeviceUserType_print;
asn_constr_check_f PersonalDeviceUserType_constraint;
ber_type_decoder_f PersonalDeviceUserType_decode_ber;
der_type_encoder_f PersonalDeviceUserType_encode_der;
xer_type_decoder_f PersonalDeviceUserType_decode_xer;
xer_type_encoder_f PersonalDeviceUserType_encode_xer;
oer_type_decoder_f PersonalDeviceUserType_decode_oer;
oer_type_encoder_f PersonalDeviceUserType_encode_oer;
per_type_decoder_f PersonalDeviceUserType_decode_uper;
per_type_encoder_f PersonalDeviceUserType_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PersonalDeviceUserType_H_ */
#include <asn_internal.h>
